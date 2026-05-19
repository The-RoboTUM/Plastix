"""Public API for the Robby control package."""

from .swerve_controller import (
    FourWIS4WIDKinematicController,
    KinematicControlDebug,
    KinematicControlResult,
    Pose2D,
    TrackingError,
    WheelReference,
    normalize_angle,
)
from .swerve_kinematic_model import (
    BodyTwist,
    FourWIS4WIDKinematicModel,
    RobotStateDerivative,
    WheelCommand,
    WheelGeometry,
    WorldTwist,
)

__all__ = [
    "BodyTwist",
    "FourWIS4WIDKinematicController",
    "FourWIS4WIDKinematicModel",
    "KinematicControlDebug",
    "KinematicControlResult",
    "Pose2D",
    "RobotStateDerivative",
    "TrackingError",
    "WheelCommand",
    "WheelGeometry",
    "WheelReference",
    "WorldTwist",
    "normalize_angle",
]
