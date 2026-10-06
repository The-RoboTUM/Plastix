"""Kinematic model for a 4WIS4WID mobile robot.

This module implements the kinematic relationships presented in:

M.-H. Lee and T.-H. S. Li, "Kinematics, dynamics and control design of
4WIS4WID mobile robots," The Journal of Engineering, 2015.

The implementation follows the paper's no-slip wheel constraints:

    v_xi = v_i cos(delta_i) = v_x - y_i * omega
    v_yi = v_i sin(delta_i) = v_y + x_i * omega

for wheel positions:

    w1 = ( a,  b)
    w2 = (-a,  b)
    w3 = (-a, -b)
    w4 = ( a, -b)

The class below provides:
- inverse kinematics: body twist -> wheel steering angles and wheel speeds
- forward kinematics: wheel steering angles and wheel speeds -> body twist
- state derivative: q_dot = J_v * u from the paper's kinematic model
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence


def _require_four(values: Sequence[float], name: str) -> None:
    if len(values) != 4:
        raise ValueError(f"{name} must contain exactly 4 values")


@dataclass(frozen=True)
class WheelGeometry:
    x: float
    y: float


@dataclass(frozen=True)
class BodyTwist:
    """Robot body twist in the robot frame."""

    vx: float
    vy: float
    omega: float


@dataclass(frozen=True)
class WorldTwist:
    """Robot posture derivative in the world frame."""

    x_dot: float
    y_dot: float
    theta_dot: float


@dataclass(frozen=True)
class WheelCommand:
    """Wheel command for one module."""

    steering_angle: float
    linear_speed: float


@dataclass(frozen=True)
class RobotStateDerivative:
    """State derivative from the paper's q_dot = J_v * u model.

    State ordering matches the paper:
        q = [x, y, theta, w1, w2, w3, w4, d1, d2, d3, d4]^T

    where:
    - x, y, theta are robot posture in the world frame
    - wi are wheel rotation angles
    - di are steering angles
    """

    x_dot: float
    y_dot: float
    theta_dot: float
    wheel_angle_rates: tuple[float, float, float, float]
    steer_angle_rates: tuple[float, float, float, float]

    def as_vector(self) -> tuple[float, ...]:
        return (
            self.x_dot,
            self.y_dot,
            self.theta_dot,
            *self.wheel_angle_rates,
            *self.steer_angle_rates,
        )


class FourWIS4WIDKinematicModel:
    """Kinematic model for a symmetric 4WIS4WID robot.

    Parameters
    ----------
    a:
        Half-wheelbase. Distance from robot centroid to the front/rear axle.
    b:
        Half-track width. Distance from robot centroid to the left/right side.
    wheel_radius:
        Common wheel radius. Used to convert between linear wheel speed and
        wheel angular speed.
    """

    def __init__(self, a: float, b: float, wheel_radius: float) -> None:
        if a <= 0.0:
            raise ValueError("a must be positive")
        if b <= 0.0:
            raise ValueError("b must be positive")
        if wheel_radius <= 0.0:
            raise ValueError("wheel_radius must be positive")

        self.a = float(a)
        self.b = float(b)
        self.wheel_radius = float(wheel_radius)
        self.modules = (
            WheelGeometry(self.a, self.b),
            WheelGeometry(-self.a, self.b),
            WheelGeometry(-self.a, -self.b),
            WheelGeometry(self.a, -self.b),
        )
        self._k = 4.0 * ((self.a * self.a) + (self.b * self.b))

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def wheel_velocity_components(self, body_twist: BodyTwist) -> tuple[tuple[float, float], ...]:
        """Return the no-slip velocity components at each wheel in the robot frame."""

        components = []
        for module in self.modules:
            vx_i = body_twist.vx - (module.y * body_twist.omega)
            vy_i = body_twist.vy + (module.x * body_twist.omega)
            components.append((vx_i, vy_i))
        return tuple(components)

    def inverse_kinematics(self, body_twist: BodyTwist) -> tuple[WheelCommand, ...]:
        """Map a desired robot-frame body twist to wheel steering and speed.

        This comes directly from the paper's wheel/body constraint equations.
        """

        commands = []
        for vx_i, vy_i in self.wheel_velocity_components(body_twist):
            steering_angle = math.atan2(vy_i, vx_i)
            linear_speed = math.hypot(vx_i, vy_i)
            commands.append(
                WheelCommand(
                    steering_angle=self._normalize_angle(steering_angle),
                    linear_speed=linear_speed,
                )
            )
        return tuple(commands)

    def inverse_kinematics_from_world_twist(
        self,
        heading: float,
        world_twist: WorldTwist,
    ) -> tuple[WheelCommand, ...]:
        """Map a world-frame posture derivative to wheel commands."""

        cos_theta = math.cos(heading)
        sin_theta = math.sin(heading)
        body_twist = BodyTwist(
            vx=(cos_theta * world_twist.x_dot) + (sin_theta * world_twist.y_dot),
            vy=(-sin_theta * world_twist.x_dot) + (cos_theta * world_twist.y_dot),
            omega=world_twist.theta_dot,
        )
        return self.inverse_kinematics(body_twist)

    def forward_kinematics_body(
        self,
        steering_angles: Sequence[float],
        wheel_linear_speeds: Sequence[float],
    ) -> BodyTwist:
        """Recover robot-frame body twist from wheel steering and linear speeds.

        For the symmetric geometry used in the paper, the pseudo-inverse has
        a closed form:

            v_x = 1/4 * sum(v_i cos(delta_i))
            v_y = 1/4 * sum(v_i sin(delta_i))
            omega = sum(W_i * v_i)

        with:

            W_i = (-y_i cos(delta_i) + x_i sin(delta_i)) / (4(a^2 + b^2))
        """

        _require_four(steering_angles, "steering_angles")
        _require_four(wheel_linear_speeds, "wheel_linear_speeds")

        vx = 0.0
        vy = 0.0
        omega = 0.0

        for module, delta_i, speed_i in zip(self.modules, steering_angles, wheel_linear_speeds):
            cos_delta = math.cos(delta_i)
            sin_delta = math.sin(delta_i)
            vx += 0.25 * speed_i * cos_delta
            vy += 0.25 * speed_i * sin_delta
            omega += ((-module.y * cos_delta) + (module.x * sin_delta)) * speed_i / self._k

        return BodyTwist(vx=vx, vy=vy, omega=omega)

    def forward_kinematics_body_from_wheel_rates(
        self,
        steering_angles: Sequence[float],
        wheel_angular_rates: Sequence[float],
    ) -> BodyTwist:
        """Recover body twist from wheel angular rates."""

        _require_four(wheel_angular_rates, "wheel_angular_rates")
        wheel_linear_speeds = tuple(rate * self.wheel_radius for rate in wheel_angular_rates)
        return self.forward_kinematics_body(steering_angles, wheel_linear_speeds)

    def forward_kinematics_world(
        self,
        heading: float,
        steering_angles: Sequence[float],
        wheel_linear_speeds: Sequence[float],
    ) -> WorldTwist:
        """Recover world-frame posture derivative from wheel states."""

        body_twist = self.forward_kinematics_body(steering_angles, wheel_linear_speeds)
        cos_theta = math.cos(heading)
        sin_theta = math.sin(heading)
        return WorldTwist(
            x_dot=(cos_theta * body_twist.vx) - (sin_theta * body_twist.vy),
            y_dot=(sin_theta * body_twist.vx) + (cos_theta * body_twist.vy),
            theta_dot=body_twist.omega,
        )

    def state_derivative(
        self,
        heading: float,
        steering_angles: Sequence[float],
        wheel_linear_speeds: Sequence[float],
        steer_rate_inputs: Sequence[float],
    ) -> RobotStateDerivative:
        """Evaluate the paper's q_dot = J_v * u kinematic model.

        Inputs
        ------
        heading:
            Robot heading theta in the world frame.
        steering_angles:
            Current wheel steering angles [d1, d2, d3, d4].
        wheel_linear_speeds:
            Wheel linear speeds [v1, v2, v3, v4].
        steer_rate_inputs:
            Steering angle rates [d1_dot, d2_dot, d3_dot, d4_dot].

        Returns
        -------
        RobotStateDerivative
            State derivative for:
            [x, y, theta, w1, w2, w3, w4, d1, d2, d3, d4]^T
        """

        _require_four(steering_angles, "steering_angles")
        _require_four(wheel_linear_speeds, "wheel_linear_speeds")
        _require_four(steer_rate_inputs, "steer_rate_inputs")

        x_dot = 0.0
        y_dot = 0.0
        theta_dot = 0.0

        for module, delta_i, speed_i in zip(self.modules, steering_angles, wheel_linear_speeds):
            global_angle = heading + delta_i
            cos_global = math.cos(global_angle)
            sin_global = math.sin(global_angle)
            x_dot += 0.25 * speed_i * cos_global
            y_dot += 0.25 * speed_i * sin_global
            cos_delta = math.cos(delta_i)
            sin_delta = math.sin(delta_i)
            theta_dot += ((-module.y * cos_delta) + (module.x * sin_delta)) * speed_i / self._k

        wheel_angle_rates = tuple(speed / self.wheel_radius for speed in wheel_linear_speeds)
        steer_angle_rates = tuple(float(rate) for rate in steer_rate_inputs)

        return RobotStateDerivative(
            x_dot=x_dot,
            y_dot=y_dot,
            theta_dot=theta_dot,
            wheel_angle_rates=wheel_angle_rates,
            steer_angle_rates=steer_angle_rates,
        )


def average_heading(angles: Iterable[float]) -> float:
    """Return the circular mean of a set of angles."""

    sin_sum = 0.0
    cos_sum = 0.0
    for angle in angles:
        sin_sum += math.sin(angle)
        cos_sum += math.cos(angle)
    return math.atan2(sin_sum, cos_sum)
