"""Object position -> robot standing pose, and approach-heading selection.

The Octopus sends the position of the *trash*, not a robot pose. Two findings
make the conversion mandatory rather than a nicety:

* ``gripperx_arm_msgs/action/PickPlastic.action`` takes ``bool execute`` - a
  fixed, blind sequence. The arm cannot be aimed.
* There is no arm or gripper link in the URDF (``gripperx_description/urdf/``):
  no kinematic model, no gripper frame.

So the robot's standing pose is the only aiming mechanism::

    robot_goal_xy  = object_xy - R(theta) * grasp_offset
    robot_goal_yaw = theta

``grasp_offset = (gx, gy)`` is the point in ``base_footprint`` where the fixed
pick sequence actually closes. IT MUST BE MEASURED, NOT DERIVED - there is no
kinematic model to derive it from. Bench procedure: place an object on a grid
relative to ``base_footprint``, run ``/pick_plastic``, record which positions
succeed; that yields both the offset and the usable tolerance. Until the OFFSET
is set this module REFUSES to resolve object goals. A plausible-looking
placeholder would produce goals that look valid, drive the robot, and miss - so
there is deliberately no default.

THE OFFSET AND THE TOLERANCE GATE DIFFERENT THINGS - USER DECISION 2026-08-19
=============================================================================
``offset_x_m``/``offset_y_m`` are the only inputs to the standoff arithmetic, so
they gate **resolution**. ``tolerance_m`` appears nowhere in that arithmetic: it
is the window used to judge "close enough to grasp" **after** arriving, so it
gates the **reached check** and nothing else. They used to be coupled through
one ``configured`` flag, which meant an unmeasured tolerance silently blocked
every goal from resolving at all - a gate at the wrong place, and one that made
auto-pick inert without saying so.

They are now separate: :attr:`GraspOffset.configured` covers resolution,
:attr:`GraspOffset.tolerance_configured` covers the reached check, and
:func:`check_reached` returns ``known=False`` while the tolerance is TO-VERIFY.
An unknown reached check must be reported loudly by the caller; it must never
fall back to a number. **This decision measured nothing.** ``offset_x_m: 0.360``
remains a user specification and ``tolerance_m`` remains TO-VERIFY.

Pure module: no rclpy, no costmap, no TF. The acceptance test for a candidate
pose is injected as a callable, which is what keeps it that way.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

#: Config sentinel. Any of these as a parameter value means "not measured yet"
#: and must leave the offset unconfigured rather than falling back to a number.
TO_VERIFY_SENTINELS = ("TO-VERIFY", "TO_VERIFY", "", "nan", "NaN", "null", "None")

#: Deviation ordering tie-break: with an even candidate count, +d and -d are
#: equidistant from the seed. We take the counter-clockwise (positive) one
#: first, purely so the choice is reproducible across runs and machines.
_CCW_FIRST = True


class GraspOffsetNotConfigured(Exception):
    """Raised when a resolution is attempted while the offset is TO-VERIFY."""

    reason = "GRASP_OFFSET_NOT_CONFIGURED"


class NoApproachCandidate(Exception):
    """Raised when no candidate heading passed the injected acceptance test."""

    reason = "NO_APPROACH_CANDIDATE"


def parse_measured_param(value: object) -> Optional[float]:
    """Config value -> float, or ``None`` when it is still TO-VERIFY.

    Accepts the sentinels above and ``None``; a non-finite number is treated as
    unset too, so a YAML ``.nan`` cannot become a silent zero.
    """
    if value is None:
        return None
    if isinstance(value, str):
        if value.strip() in TO_VERIFY_SENTINELS:
            return None
        try:
            value = float(value)
        except ValueError:
            return None
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    return None


@dataclass(frozen=True)
class GraspOffset:
    """The measured grasp point in ``base_footprint``.

    ``configured`` is false whenever either OFFSET component is still TO-VERIFY;
    the tolerance is deliberately not part of it (see the module docstring).
    Use :meth:`require` at every use site; never read ``x``/``y`` without it.
    """

    x: Optional[float] = None
    y: Optional[float] = None
    tolerance_m: Optional[float] = None

    @classmethod
    def from_params(
        cls, offset_x: object, offset_y: object, tolerance: object
    ) -> "GraspOffset":
        return cls(
            x=parse_measured_param(offset_x),
            y=parse_measured_param(offset_y),
            tolerance_m=parse_measured_param(tolerance),
        )

    @property
    def configured(self) -> bool:
        """Enough to RESOLVE a goal: the offset only. Not the tolerance."""
        return self.x is not None and self.y is not None

    @property
    def tolerance_configured(self) -> bool:
        """Enough to judge REACHED. Independent of :attr:`configured`."""
        return self.tolerance_m is not None and self.tolerance_m > 0.0

    @property
    def standoff_m(self) -> Optional[float]:
        """Distance the robot origin ends up from the object. Derived, not
        configured: it is just the length of the offset vector."""
        if self.x is None or self.y is None:
            return None
        return math.hypot(self.x, self.y)

    def require(self) -> Tuple[float, float]:
        if not self.configured:
            raise GraspOffsetNotConfigured(
                "grasp.offset_x_m / offset_y_m are TO-VERIFY (bench measurement "
                "on the real robot); refusing to resolve an object goal rather "
                "than guessing the grasp point"
            )
        assert self.x is not None and self.y is not None
        return self.x, self.y


def normalize_angle(angle: float) -> float:
    """Wrap to (-pi, pi]."""
    wrapped = math.remainder(angle, math.tau)
    # math.remainder yields -pi for exactly half a turn; keep +pi so that a
    # 180 deg deviation sorts as the last candidate rather than the first.
    return math.pi if wrapped == -math.pi else wrapped


def bearing(from_xy: Sequence[float], to_xy: Sequence[float]) -> float:
    """Heading from one point to another, in the map frame."""
    return math.atan2(to_xy[1] - from_xy[1], to_xy[0] - from_xy[0])


def robot_pose_for(
    object_xy: Sequence[float], theta: float, offset: GraspOffset
) -> Tuple[float, float, float]:
    """``object_xy - R(theta) * grasp_offset`` and yaw ``theta``.

    Raises :class:`GraspOffsetNotConfigured` while the offset is TO-VERIFY.
    """
    gx, gy = offset.require()
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    x = object_xy[0] - (cos_t * gx - sin_t * gy)
    y = object_xy[1] - (sin_t * gx + cos_t * gy)
    return x, y, normalize_angle(theta)


def grasp_point_for(
    robot_xy: Sequence[float], yaw: float, offset: GraspOffset
) -> Tuple[float, float]:
    """Forward direction: where the fixed pick sequence closes, given a pose.

    Only used by tests and the RViz preview, but it is the definition
    :func:`robot_pose_for` inverts, so it lives next to it.
    """
    gx, gy = offset.require()
    cos_t, sin_t = math.cos(yaw), math.sin(yaw)
    return (
        robot_xy[0] + cos_t * gx - sin_t * gy,
        robot_xy[1] + sin_t * gx + cos_t * gy,
    )


def candidate_headings(seed_theta: float, count: int) -> List[float]:
    """``count`` headings spread over the full ring, ordered by deviation.

    The first entry is the seed itself, then the ring is walked outwards by
    increasing angular deviation: seed, seed+step, seed-step, seed+2*step, ...
    The normal case is "drive at it and stop short" and stops at the first
    entry; the ring only matters when a wall sits behind the object.
    """
    if count < 1:
        raise ValueError("count must be >= 1")
    step = math.tau / count
    deviations = [normalize_angle(k * step) for k in range(count)]
    deviations.sort(key=lambda d: (abs(d), -d if _CCW_FIRST else d))
    return [normalize_angle(seed_theta + d) for d in deviations]


@dataclass(frozen=True)
class Candidate:
    """One approach hypothesis handed to the injected acceptance test."""

    index: int
    theta: float
    deviation_rad: float
    robot_x: float
    robot_y: float
    object_x: float
    object_y: float


@dataclass(frozen=True)
class CandidateVerdict:
    """Result of the injected acceptance test for one candidate."""

    accepted: bool
    reason: str = ""
    detail: str = ""
    warnings: Tuple[str, ...] = ()


@dataclass(frozen=True)
class ApproachResult:
    chosen: Optional[Candidate]
    #: ``(candidate, verdict)`` for every candidate examined, in the order they
    #: were examined. Kept for the preview and the rejection report: "no
    #: approach found" is useless to an operator without the per-candidate
    #: reasons.
    evaluated: Tuple[Tuple[Candidate, CandidateVerdict], ...] = ()
    warnings: Tuple[str, ...] = ()

    @property
    def ok(self) -> bool:
        return self.chosen is not None

    def failure_reasons(self) -> List[str]:
        return [v.reason for _, v in self.evaluated if not v.accepted]


AcceptFn = Callable[[Candidate], CandidateVerdict]


def choose_approach(
    object_xy: Sequence[float],
    robot_xy: Sequence[float],
    offset: GraspOffset,
    accept: AcceptFn,
    candidate_count: int = 12,
    seed_theta: Optional[float] = None,
) -> ApproachResult:
    """Pick the least-deviating acceptable approach heading.

    ``seed_theta`` defaults to the bearing from the current robot pose to the
    object; pass it explicitly to honour an operator override. ``accept`` gets a
    fully resolved :class:`Candidate` and decides using whatever it likes
    (geofence, costmap, ``compute_path_to_pose``) - this module stays free of
    all of it.

    Raises :class:`GraspOffsetNotConfigured` if the offset is TO-VERIFY, so an
    unmeasured robot cannot produce a "valid" goal.
    """
    offset.require()
    if seed_theta is None:
        seed_theta = bearing(robot_xy, object_xy)
    evaluated: List[Tuple[Candidate, CandidateVerdict]] = []
    warnings: List[str] = []
    for index, theta in enumerate(candidate_headings(seed_theta, candidate_count)):
        x, y, yaw = robot_pose_for(object_xy, theta, offset)
        candidate = Candidate(
            index=index,
            theta=yaw,
            deviation_rad=normalize_angle(theta - seed_theta),
            robot_x=x,
            robot_y=y,
            object_x=float(object_xy[0]),
            object_y=float(object_xy[1]),
        )
        verdict = accept(candidate)
        evaluated.append((candidate, verdict))
        warnings.extend(verdict.warnings)
        if verdict.accepted:
            return ApproachResult(candidate, tuple(evaluated), tuple(warnings))
    return ApproachResult(None, tuple(evaluated), tuple(warnings))


# ---------------------------------------------------------------------------
# the reached check - the ONLY thing grasp.tolerance_m gates
# ---------------------------------------------------------------------------
#: Returned as ``reason`` while ``grasp.tolerance_m`` is TO-VERIFY. It is a
#: named, reported state and never a silent pass: the caller must surface it.
TOLERANCE_NOT_CONFIGURED = "GRASP_TOLERANCE_NOT_CONFIGURED"


@dataclass(frozen=True)
class ReachedVerdict:
    """Did the robot actually stop close enough for the blind pick to work?

    ``known`` is the whole point of this type. With ``grasp.tolerance_m``
    TO-VERIFY there is no measured window to compare against, so the honest
    answer is "unknown" - not "yes" and not "no". Defaulting it either way would
    be a fabricated measurement (FR-12 item 8): "yes" would licence a blind pick
    on a number nobody measured, "no" would make every arrival a failure and
    blacklist every target after two attempts.
    """

    known: bool
    reached: bool
    distance_m: float
    reason: str = ""
    detail: str = ""


def check_reached(
    object_xy: Sequence[float],
    robot_pose: Sequence[float],
    offset: GraspOffset,
) -> ReachedVerdict:
    """Distance from the grasp point to the object, judged against the tolerance.

    Requires the OFFSET (to know where the grasp point is) but only *reports*
    against the TOLERANCE. With the tolerance unmeasured the distance is still
    computed and returned - it is real information the operator wants - and only
    the verdict is withheld.
    """
    # The existing forward definition, deliberately reused rather than
    # restated: after arrival the question is where the grasp point landed for
    # the pose the robot ACTUALLY reached, and Nav2 stops within
    # xy_goal_tolerance / yaw_goal_tolerance of the pose it was given, not on it.
    grasp_x, grasp_y = grasp_point_for(
        (robot_pose[0], robot_pose[1]), float(robot_pose[2]), offset
    )
    distance = math.hypot(float(object_xy[0]) - grasp_x, float(object_xy[1]) - grasp_y)
    if not offset.tolerance_configured:
        return ReachedVerdict(
            known=False,
            reached=False,
            distance_m=distance,
            reason=TOLERANCE_NOT_CONFIGURED,
            detail=(
                f"grasp point is {distance:.3f} m from the object, but "
                "grasp.tolerance_m is TO-VERIFY so there is no measured window "
                "to judge it against. The bench procedure that yields the "
                "offset yields this too"
            ),
        )
    assert offset.tolerance_m is not None
    reached = distance <= offset.tolerance_m
    return ReachedVerdict(
        known=True,
        reached=reached,
        distance_m=distance,
        reason="" if reached else "GRASP_POINT_OUT_OF_TOLERANCE",
        detail=(
            f"grasp point is {distance:.3f} m from the object "
            f"(tolerance {offset.tolerance_m:.3f} m)"
        ),
    )
